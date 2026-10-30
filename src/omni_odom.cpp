//
// Created by root on 2026/9/22.
//

#include "../include/omni_odom.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <Eigen/Dense>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Imu.h>


OmniOdom::OmniOdom(double init_x, double init_y, double init_yaw) : x_(init_x), y_(init_y), yaw_(init_yaw) {
    odom_tf_.header.frame_id = "odom";
    odom_tf_.child_frame_id = "base_link";
}

void OmniOdom::init() {
    imu_sub = nh.subscribe(imu_topic, 1, &OmniOdom::imu_callback, this);
}

void OmniOdom::publishTf(const ros::Time &time) {
    odom_tf_.header.stamp = time;

    odom_tf_.transform.translation.x = x_;
    odom_tf_.transform.translation.y = y_;
    odom_tf_.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom_tf_.transform.rotation.x = q.x();
    odom_tf_.transform.rotation.y = q.y();
    odom_tf_.transform.rotation.z = q.z();
    odom_tf_.transform.rotation.w = q.w();

    tf_broadcaster_.sendTransform(odom_tf_);
}

//正运动学 输入为各个轮子相对地面速度d_v,输出车相对地面速度
void omni_kinematics_solution(double chassis_radius, const std::vector<double> &wheel_angle,
                              const std::vector<double> &d_v, double &result_x, double &result_y, double &result_yaw) {
    size_t n = wheel_angle.size();
    if (n != d_v.size()) {
        ROS_ERROR("[omni_kinematics_solution]quantity mismatch: wheel_angle：%lu d_v: %lu ", n, d_v.size());
        return;
    }
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd offset(n);
    for (Eigen::Index i = 0; i < A.rows(); ++i) {
        double theta = wheel_angle[i];
        A(i, 0) = std::sin(theta);
        A(i, 1) = std::cos(theta);
        A(i, 2) = chassis_radius;
        offset(i) = d_v[i];
    }
    Eigen::Vector3d twist;
    if (n == 3) {
        twist = A.colPivHouseholderQr().solve(offset);
    } else {
        twist = (A.transpose() * A).ldlt().solve(A.transpose() * offset);
    }
    result_x = twist(0);
    result_y = twist(1);
    result_yaw = twist(2);
}

void OmniOdom::update(double chassis_radius, double wheel_radius,
                      const std::vector<hardware_interface::JointHandle> &joint_handles,
                      std::vector<double> &wheel_angle, const ros::Time &time) {
    const size_t n = joint_handles.size();
    if (n == 0) return;

    if (joint_state_last.size() != n) {
        joint_state_last.resize(n);
        for (size_t i = 0; i < n; ++i) {
            joint_state_last[i] = joint_handles[i].getPosition();
        }
        return; // 第一周期不积分
    }

    //正运动学
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd offset(n);
    for (Eigen::Index i = 0; i < A.rows(); ++i) {
        double theta = wheel_angle[i];
        A(i, 0) = std::sin(theta);
        A(i, 1) = std::cos(theta);
        A(i, 2) = chassis_radius;

        offset(i) = (joint_handles[i].getPosition() - joint_state_last[i]) * wheel_radius;
        joint_state_last[i] = joint_handles[i].getPosition();
    }
    Eigen::Vector3d twist;
    if (n == 3) {
        twist = A.colPivHouseholderQr().solve(offset);
    } else {
        twist = (A.transpose() * A).ldlt().solve(A.transpose() * offset);
    }
    double psi_mid = yaw_ + 0.5 * twist(2); // 本周期中点朝向
    double c = std::cos(psi_mid), s = std::sin(psi_mid);
    x_ += c * twist(0) - s * twist(1);
    y_ += s * twist(0) + c * twist(1);
    yaw_ += twist(2);
    double odom_yaw_d = std::atan2(std::sin(yaw_), std::cos(yaw_)); //神秘归一化技巧


    if (imu_filter_en) {
        // 加锁保护，读取回调里积分的 IMU 数据
        std::lock_guard<std::mutex> lock(imu_mutex_);

        if (imu_valid_) {
            // double err = odom_yaw_ - imu_yaw_;
            // while (err > M_PI) err -= 2.0 * M_PI;
            // while (err < -M_PI) err += 2.0 * M_PI;

            // imu_yaw_ += 0.01 * err;

            yaw_ = imu_yaw_;
        } else {
            // 如果 IMU 还没收到数据，先纯靠里程计
            yaw_ += odom_yaw_d;
        }
    } else {
        // 如果没有开启 IMU 互补滤波，直接使用里程计
        yaw_ += odom_yaw_d;
    }


    publishTf(time);
}

void OmniOdom::reset() {
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    joint_state_last.clear(); // 清空，下次 update 重新初始化
}

bool OmniOdom::resetCallback(std_srvs::Trigger::Request &req,
                             std_srvs::Trigger::Response &res) {
    (void) req; // 没用请求参数
    reset();
    res.success = true;
    res.message = "odometry reset to (0, 0, 0)";
    ROS_INFO("[OmniOdom] odometry reset by service call");
    return true;
}

void OmniOdom::imu_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (msg->orientation_covariance[0] >= 0.0) {
        tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
        q.normalize();
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        imu_yaw_ = yaw;
    } else {
        double current_time = msg->header.stamp.toSec();
        if (imu_last_time_ == 0.0) {
            imu_last_time_ = current_time;
            return; // 第一帧数据只记录时间，不积分
        }

        // 2. 计算时间差 dt
        double dt = current_time - imu_last_time_;
        // 如果 dt 过大（比如程序卡顿、或者刚刚复位），则丢弃这一帧，防止积分突变
        if (dt <= 0.0 || dt > 0.1) {
            imu_last_time_ = current_time;
            return;
        }
        imu_last_time_ = current_time;

        double raw_gyro_z = msg->angular_velocity.z;
        const double alpha_lpf = 0.2;
        last_gyro_z_ = (1.0 - alpha_lpf) * last_gyro_z_ + alpha_lpf * raw_gyro_z;

        // 4. 零偏(Bias)在线估计
        gyro_z_lpf_ = 0.999 * gyro_z_lpf_ + 0.001 * last_gyro_z_;
        if (std::abs(last_gyro_z_ - gyro_z_lpf_) < 0.01) {
            gyro_bias_z_ += 0.0005 * (gyro_z_lpf_ - gyro_bias_z_);
        }

        // 5. 减去零偏
        double clean_gyro_z = last_gyro_z_ - gyro_bias_z_;

        // 6. 积分得到 IMU 的偏航角
        imu_yaw_ += clean_gyro_z * dt;

        // 7. 限制范围 (-pi 到 pi)，防止数值过大
        if (imu_yaw_ > M_PI) {
            imu_yaw_ -= 2.0 * M_PI;
        } else if (imu_yaw_ < -M_PI) {
            imu_yaw_ += 2.0 * M_PI;
        }
    }

    imu_valid_ = true;
}
