//
// Created by root on 2026/11/3.
//

#ifndef SRC_HERO_GIMBAL_CONTROLLER_H
#define SRC_HERO_GIMBAL_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <tf2_ros/transform_listener.h>
#include <rm_msgs_exam/GimbalCmd.h>
#include <sensor_msgs/Imu.h>
#include <memory.h>

namespace hero_gimbal_controller {
#define PI 3.14159265358979323846

  class Gimbal_joint {
  public:
    Gimbal_joint() = default;

    hardware_interface::JointHandle joint_;
    control_toolbox::Pid vel_pid_;
    control_toolbox::Pid pos_pid_;
    int state = 0;
    /*
     *0 = 为停止
     *1 = 力控制
     *2 = 速度控制
     */
    void setVel(double v);

    void setTor(double tor);

    void setPos(double tor);

    void setTorBias(double tor);

    double getVel_target() const { return vel_target_; }
    double getTor_target() const { return tor_target_; }
    double getPos_target() const { return pos_target_; }

    void update(ros::Duration dt);


    double world_pos = 0;

  private:
    double rad_max_limit = INFINITY;
    double tor_max_limit = INFINITY;

    double pos_target_ = 0;
    double vel_target_ = 0;
    double tor_target_ = 0;
    double tor_bias_ = 0;
  };


  class HeroGimbalController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
  public:
    HeroGimbalController() = default;

    ~HeroGimbalController() override = default;

    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
              ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void starting(const ros::Time &time) override;

    // void stopping(const ros::Time &time) override;


  private:
    int state_{};
    ros::Time last_change_;

    ros::Time last_cmd_time_;

    ros::Subscriber gimbal_cmd_sub_;
    rm_msgs_exam::GimbalCmd gimbal_cmd_msg_;
    ros::Subscriber imu_sub_;
    sensor_msgs::Imu imu_mssg;
    ros::Subscriber debug_sub_;

    ros::Publisher debug_pub_;
    std_msgs::Float64MultiArray debug_pub_msgs;

    Gimbal_joint yaw_joint, pitch_joint;
    // double

    void gimbalCmdCallback(const rm_msgs_exam::GimbalCmd::ConstPtr &msg);

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void debugCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);
  };
} // namespace hero_gimbal_controller

#endif //SRC_HERO_GIMBAL_CONTROLLER_H
