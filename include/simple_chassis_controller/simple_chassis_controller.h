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


namespace simple_chassis_controller {

    class Wheel_joint {
        public:
        hardware_interface::JointHandle joint_handle_;
        double vel_target_ = 0;
        double vel_actual_ = 0;
        double tor_target_ = 0;
        double tor_actual_ = 0;

        control_toolbox::Pid Pid_;
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

        hardware_interface::JointHandle front_left_joint_, front_right_joint_, back_left_joint_, back_right_joint_;
        double front_left_vel_target, front_right_vel_target, back_left_vel_target, back_right_vel_target;

    private:
        int state_{};
        ros::Time last_change_;

        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber joint_state_sub_;
        ros::Time last_cmd_time_;
        ros::Subscriber debug_sub_;
        geometry_msgs::Twist cmd_vel_target;
        geometry_msgs::Twist cmd_vel_;

        double wheel_base = 0;

        void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
        // void jointStateCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void debugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

        // void computeWheelVelocities(const geometry_msgs::Twist &twist);

        bool force_col = false;
        bool odom_pub_en = false;

        double wheel_radius_ = 0.08;
        double cmd_vel_timeout_ = 0.5;     // cmd_vel 超时（秒）
        double max_wheel_velocity_ = 100.0; // 轮子最大角速度 (rad/s)


    };
} // namespace simple_chassis_controller

#endif //SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
