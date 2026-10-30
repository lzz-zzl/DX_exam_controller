//
// Created by root on 2026/9/21.
//
#include <string>
#include <vector>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <cmath>
#include <Eigen/Dense>


enum robo_state_e {
    quit_st = 0,
    chassis_free_st,
    follow_gimbal_st,
    spining_top_st,
};

robo_state_e robo_state = chassis_free_st;

double chassis_pitch = 0;
double chassis_roll = 0;
double standard_gravity = 265;


double F_x = 0;
double F_y = 0;
double T_yaw = 0;


double vel_x_rela = 0;
double vel_y_rela = 0;
double vel_az_rela = 0;

double rf_joint_rad = 0;

size_t num_wheels_ = 0;
std::vector<std::string> wheel_names;
std::vector<double> wheels_vel;
double chassis_radius = 0;
double wheel_radius;
std::vector<double> wheel_angles;

constexpr double linear_max_acceleration = 3.0;
constexpr double angular_max_acceleration = 8.0;
ros::Time last_cb_time_;

void acceleration_limit(double linear_x, double linear_y, double angular_z) {
    ros::Time now = ros::Time::now();
    double dt = (now - last_cb_time_).toSec();;
    last_cb_time_ = now;

    double dVx = linear_x - vel_x_rela;
    double dVy = linear_y - vel_y_rela;
    double dVLinear_sq = dVx * dVx + dVy * dVy;
    double accLinear_sq = linear_max_acceleration * linear_max_acceleration * dt * dt;
    if (dVLinear_sq > accLinear_sq) {
        double scale = std::sqrt(accLinear_sq / dVLinear_sq);
        dVx *= scale;
        dVy *= scale;
        vel_x_rela += dVx;
        vel_y_rela += dVy;
    } else {
        vel_x_rela = linear_x;
        vel_y_rela = linear_y;
    }

    double dWz = angular_z - vel_az_rela;
    double dWz_max = angular_max_acceleration * dt;
    if (dWz > dWz_max) {
        vel_az_rela += dWz_max;
    } else if (dWz < -dWz_max) {
        vel_az_rela -= dWz_max;
    } else {
        vel_az_rela = angular_z;
    }
}


#define PI 3.14159265358979323846

void cmd_callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    if (msg->data[8] != robo_state) {
        robo_state = static_cast<robo_state_e>(msg->data[8]);
        ROS_INFO("[standard_cmd_handle]state:%d ", robo_state);
    }
    double temp_vel_x_rela = msg->data[2];
    double temp_vel_y_rela = msg->data[3];
    double temp_vel_az_rela = msg->data[4];
    //加速度限制
    acceleration_limit(temp_vel_x_rela, temp_vel_y_rela, temp_vel_az_rela);
}

//test
double temp_test = 0;
//~test
void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg) {
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // ROS_INFO("yaw_world = %.4f rad (%.2f deg)", pitch, yaw * 180.0 / M_PI);
}

bool chassis_force_col = false;

bool read_controlCfg(const ros::NodeHandle &nh) {
    ROS_INFO("[standard_cmd_handle] chassis_force_col = %d", chassis_force_col);
    if (!nh.getParam("standard_controller/wheel_joints", wheel_names)) {
        ROS_ERROR("[standard_cmd_handle] wheel_joints no found");
        return false;
    }
    nh.param("wheel_radius", wheel_radius, 0.05);


    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "standard_cmd_handle");
    ros::NodeHandle nh;
    read_controlCfg(nh);

    ros::Publisher pub_classis_vel = nh.advertise<geometry_msgs::Twist>("/standard_controller/cmd_vel", 1);
    ros::Publisher pub_classis_force = nh.advertise<std_msgs::Float64MultiArray>(
        "/standard_controller/chassis/force", 1);
    ros::Publisher yaw_pub = nh.advertise<std_msgs::Float64>("/standard_controller/gimbal/yaw_torque", 1);
    ros::Publisher pitch_pub = nh.advertise<std_msgs::Float64>("/standard_controller/gimbal/pitch_torque", 1);
    ros::Publisher debug_pub = nh.advertise<std_msgs::Float64MultiArray>("/standard_controller/debug", 1);

    ros::Subscriber cmd_sub = nh.subscribe("/standard_cmd", 1, cmd_callback);
    ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 1, joint_states_callback);
    ros::Subscriber imu_sub = nh.subscribe("/gimbal_imu/data_filtered", 1, imu_callback);

    geometry_msgs::Twist cmd_vel_msg;
    std_msgs::Float64MultiArray cmd_T_msg;

    double loop_frq = 400;
    ros::Rate loop_rate(loop_frq);
    while (ros::ok()) {
        ros::spinOnce();

        //底盘
        cmd_vel_msg.linear.x = vel_x_rela;
        cmd_vel_msg.linear.y = vel_y_rela;
        cmd_vel_msg.angular.z = vel_az_rela;
        pub_classis_vel.publish(cmd_vel_msg);

        loop_rate.sleep();
    }
}
