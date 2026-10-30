//
// Created by qiayuan on 2/6/21.
//

#include "simple_chassis_controller/simple_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace simple_chassis_controller {
    bool SimpleChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                       ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        front_left_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

        controller_nh.param("wheel_base", wheel_base, 5.0);


        cmd_vel_sub_ = controller_nh.subscribe<geometry_msgs::Twist>(
            "cmd_vel", 1, &SimpleChassisController::cmdVelCallback, this);
        debug_sub_ = controller_nh.subscribe(
                    "debug", 1, &SimpleChassisController::debugCallback, this);

        ROS_INFO("[SimpleChassisController] 初始化成功");
        return true;
    }

    void SimpleChassisController::starting(const ros::Time & /*time*/) {

    }

    void SimpleChassisController::update(const ros::Time &time, const ros::Duration &period) {
        if (force_col) {

        }else {
            front_left_vel_target  = cmd_vel_target.linear.x - cmd_vel_target.linear.y - cmd_vel_target.angular.z*wheel_base;
            front_right_vel_target = cmd_vel_target.linear.x + cmd_vel_target.linear.y - cmd_vel_target.angular.z*wheel_base;
            back_left_vel_target   = cmd_vel_target.linear.x + cmd_vel_target.linear.y + cmd_vel_target.angular.z*wheel_base;
            back_right_vel_target  = cmd_vel_target.linear.x - cmd_vel_target.linear.y + cmd_vel_target.angular.z*wheel_base;

        }



    }

    void SimpleChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        cmd_vel_target = *msg;
        last_cmd_time_ = ros::Time::now();
    }


}

PLUGINLIB_EXPORT_CLASS(simple_chassis_controller::SimpleChassisController, controller_interface::ControllerBase)

