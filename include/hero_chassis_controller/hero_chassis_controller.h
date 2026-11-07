//
// Created by qiayuan on 2/6/21.
//

#ifndef HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/buffer.h>

namespace hero_chassis_controller
{
#define PI 3.14159265358979323846

struct Command
{
  geometry_msgs::Twist cmd_vel_;
  // rm_msgs_exam::ChassisCmd cmd_chassis_;
  std_msgs::Float64MultiArray debug_msg_;
  ros::Time stamp_;
};

class Wheel_joint
{
public:
  Wheel_joint() = default;

  hardware_interface::JointHandle joint_;
  control_toolbox::Pid pid_;
  int state = 0;
  /*
   *0 = 为停止
   *1 = 力控制
   *2 = 速度控制
   */
  void setVel(double v);

  void setTor(double tor);

  void setTorBias(double tor);

  double getVel_target() const { return vel_target_; }
  double getTor_target() const { return tor_target_; }

  void update(ros::Duration dt);

private:
  double vel_target_ = 0;
  double tor_target_ = 0;
  double tor_bias_ = 0;
};

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController() = default;

  ~HeroChassisController() override = default;

  // static void acceleration_limit(double wish_vel_x, double wish_vel_y, double wish_rad_z, double dt,
  //                                double& cmd_vel_x,
  //                                double& cmd_vel_y, double& cmd_rad_z);

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface,
            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void starting(const ros::Time& time) override;

  // void stopping(const ros::Time &time) override;

  Wheel_joint front_left_joint_, front_right_joint_
              , back_left_joint_, back_right_joint_;
  double init_kp_ = 4, init_ki_ = 0.1, init_kd_ = 0, init_i_limit_ = 10;

private:
  int state_{ RAW };
  bool state_changed_ = false;
  ros::Time last_change_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber debug_sub_;
  ros::Subscriber joint_state_sub_;

  ros::Publisher debug_pub_;

  Command cmd_struct_{};
  realtime_tools::RealtimeBuffer<Command> cmd_rt_buffer_;
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void debugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);

  std_msgs::Float64MultiArray debug_pub_msgs;

  geometry_msgs::Twist cmd_vel_;         //初始命令速度
  geometry_msgs::Twist chassis_cmd_vel_; // 确保在地面坐标系下加速度限制

  //地盘
  double wheel_base = 0.4;
  double wheel_track = 0.4;
  double scale_yaw = (wheel_base + wheel_track) / 2;
  double chassis_vx_ = 0, chassis_vy_ = 0, chassis_wz_ = 0;

  bool global_vel_cmd_en = false;
  bool odom_pub_en = false;

  double max_effort_ = 100;
  double wheel_radius_ = 0.07625;
  double cmd_vel_timeout_ = 0.5;      // cmd_vel 超时（秒）
  double max_wheel_velocity_ = 100.0; // 轮子最大角速度 (rad/s)

  double rale_gimbal_yaw_pos = 0;
  void stateRaw();
  void stateWorld();
  void stateFollow(ros::Duration period);
  void stateTwist();
  void stateSinSpin(ros::Duration period);

  void rotateCmdVel(const geometry_msgs::Twist& origin, geometry_msgs::Twist& output, double yaw);
  void chassis_acc_limit(const geometry_msgs::Twist& origin, geometry_msgs::Twist& output, double chassis_wz,
                         ros::Duration period);
  //tfS
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::TransformStamped odom_tf_;
  double chassis_x_ = 0.0;
  double chassis_y_ = 0.0;
  double chassis_yaw_ = 0.0;
  void updateOdom(const ros::Time& time, const ros::Duration& period);

  void publishTf(const ros::Time& time);

  enum
  {
    RAW = 0,
    FOLLOW = 1,
    TWIST = 2,
    WORLD = 3,
    SIN_SPIN = 4,
  };

  control_toolbox::Pid follow_pid_;
};
} // namespace hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H