//
// Created by qiayuan on 2/6/21.
//

#ifndef SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
#define SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <sensor_msgs/JointState.h>


namespace simple_chassis_controller {
    class Wheel_joint {
    public:
        Wheel_joint() = default;

        hardware_interface::JointHandle joint_handle_;
        control_toolbox::Pid pid_;
        int state = 0;
        /*
         *0 = 为停止
         *1 = 力控制
         *2 = 速度控制
         */
        void setVel(double v);

        void setTor(double tor);

        double getVel_target() const { return vel_target_; }
        double getTor_target() const { return tor_target_; }

        void update(ros::Duration dt);

    private:
        double vel_target_ = 0;
        double tor_target_ = 0;
    };

    class SimpleChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
    public:
        SimpleChassisController() = default;

        ~SimpleChassisController() override = default;

        bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
                  ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

        // void stopping(const ros::Time &time) override;

        Wheel_joint front_left_joint_, front_right_joint_
                , back_left_joint_, back_right_joint_;
        double init_kp_ = 4, init_ki_ = 0.1, init_kd_ = 0, init_i_limit_ = 10;

    private:
        int state_{};
        ros::Time last_change_;

        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber debug_sub_;

        ros::Publisher debug_pub_;
        std_msgs::Float64MultiArray debug_pub_msgs;

        ros::Time last_cmd_time_;
        double cmd_vel_x = 0, cmd_vel_y = 0, cmd_rad_z = 0;
        geometry_msgs::Twist cmd_vel_;

        double wheel_base = 0.4;
        double wheel_track = 0.4;

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

        void debugCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

        // void computeWheelVelocities(const geometry_msgs::Twist &twist);

        bool force_col = false;
        bool odom_pub_en = false;

        double max_effort_ = 100;
        double wheel_radius_ = 0.07625;
        double cmd_vel_timeout_ = 0.5; // cmd_vel 超时（秒）
        double max_wheel_velocity_ = 100.0; // 轮子最大角速度 (rad/s)
    };
} // namespace simple_chassis_controller

#endif //SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
