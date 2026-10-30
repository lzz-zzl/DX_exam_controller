//
// Created by root on 2026/9/22.
//

#ifndef SRC_Mecanum_ODOM_H
#define SRC_Mecanum_ODOM_H

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <hardware_interface/joint_command_interface.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/Imu.h>
#include <mutex>

class MecanumOdom {
public:
    bool imu_filter_en = false;
    std::string imu_topic;

    void init();

    MecanumOdom(double init_x = 0, double init_y = 0, double init_yaw = 0);

    void update(double chassis_radius, double chawheel_radius,
                const std::vector<hardware_interface::JointHandle> &joint_handles,
                std::vector<double> &wheel_angle_, const ros::Time &time
    );

    void publishTf(const ros::Time &time);

    void reset();

    bool resetCallback(std_srvs::Trigger::Request &req,
                       std_srvs::Trigger::Response &res);

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg);

private:
    bool imu_valid_ = false;
    std::mutex imu_mutex_;
    ros::NodeHandle nh;
    ros::Subscriber imu_sub;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::TransformStamped odom_tf_;


    double chassis_radius_ = 0.0; // 底盘半径（轮子安装圆半径）
    double wheel_radius_ = 0.08; // 轮子半径

    int num_wheels_ = 0;
    std::vector<double> wheel_angle_; // 每个轮子的安装角度
    std::vector<double> joint_state_last;


    double init_x_ = 0.0;
    double init_y_ = 0.0;
    double init_z_ = 0.0;
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    //imu_complementary
    double imu_last_time_ = 0.0;
    double imu_yaw_ = 0.0;      // 纯角速度积分值 (弧度)
    double last_gyro_z_ = 0.0;

    double gyro_bias_z_ = 0.0;
    double gyro_z_lpf_ = 0.0;         // 角速度低通滤波值 (用于静止检测)
};


#endif //SRC_Mecanum_ODOM_H
