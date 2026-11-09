//
// Created by root on 2026/11/3.
//

#include "../include/hero_gimbal_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>

namespace hero_gimbal_controller
{
//test
double test[5];
//test end

void Gimbal_joint::setTor(double tor)
{
  state = 1;
  tor_target_ = tor;
}

void Gimbal_joint::setVel(double v)
{
  state = 2;
  vel_target_ = v;
}

void Gimbal_joint::setPos(double pos)
{
  state = 3;
  pos_target_ = pos;
}

void Gimbal_joint::setTorBias(double tor)
{
  tor_bias_ = tor;
}


void Gimbal_joint::update(ros::Duration period)
{
  if (state == 0)
  {
    joint_.setCommand(0);
  }
  else if (state == 1)
  {
    joint_.setCommand(tor_target_);
  }
  else if (state == 2)
  {
    double err = vel_target_ - world_vel;
    joint_.setCommand(vel_pid_.computeCommand(err, period) + tor_bias_);
  }
  else if (state == 3)
  {
    double err = pos_target_ - world_pos;
    err = std::remainder(err, 2 * PI);
    double p_err = pos_pid_.computeCommand(err, period) - world_vel;
    joint_.setCommand(vel_pid_.computeCommand(p_err, period) + tor_bias_);
  }
}

PID* debug_pid = nullptr;
// control_toolbox::Pid* debug_pid = nullptr;

bool HeroGimbalController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  controller_nh.param("gimbal_cmd_timeout", gimbal_cmd_timeout_, 0.5);
  controller_nh.param("mass_origin_x", mass_origin_.x, 0.0);
  controller_nh.param("mass_origin_z", mass_origin_.z, 0.0);
  controller_nh.param("gravity_", gravity_, 9.81);
  controller_nh.param("gimbal_mass", gimbal_mass_, 0.0);
  controller_nh.param("enable_gravity_compensation_", enable_gravity_compensation_, false);

  double init_yaw_kp_{}, init_yaw_ki_{}, init_yaw_kd_{}, int_yaw_i_limit_{},
         init_pitch_kp_{}, init_pitch_ki_{}, init_pitch_kd_{}, int_pitch_i_limit_{};
  controller_nh.param("pid_yaw_kp", init_yaw_kp_, 7.0);
  controller_nh.param("pid_yaw_ki", init_yaw_ki_, 1.0);
  controller_nh.param("pid_yaw_kd", init_yaw_kd_, 0.0);
  controller_nh.param("int_yaw_i_limit", int_yaw_i_limit_, 2.0);
  controller_nh.param("pid_pitch_kp", init_pitch_kp_, 8.0);
  controller_nh.param("pid_pitch_ki", init_pitch_ki_, 1.0);
  controller_nh.param("pid_pitch_kd", init_pitch_kd_, 0.0);
  controller_nh.param("int_pitch_i_limit", int_pitch_i_limit_, 2.0);
  debug_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("/hero_gimbal_controller/pub_debug", 1);
  gimbal_cmd_sub_ = controller_nh.subscribe("/controller/hero_gimbal_controller/cmd", 1,
                                            &HeroGimbalController::gimbalCmdCallback,
                                            this);
  imu_sub_ = controller_nh.subscribe("/gimbal_imu/data_filtered", 1, &HeroGimbalController::imuCallback, this);
  debug_sub_ = controller_nh.subscribe("/hero_gimbal_controller/sub_debug", 1, &HeroGimbalController::debugCallback,
                                       this);

  try
  {
    pitch_joint_.joint_ = effort_joint_interface->getHandle("pitch_joint");
    yaw_joint_.joint_ = effort_joint_interface->getHandle("yaw_joint");
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR("[HeroGimbalController] getHandle failed: %s", e.what());
    return false;
  }
  yaw_joint_.pos_pid_.setGains(init_yaw_kp_, init_yaw_ki_, init_yaw_kd_, int_yaw_i_limit_, -int_yaw_i_limit_);
  pitch_joint_.pos_pid_.setGains(init_pitch_kp_, init_pitch_ki_, init_pitch_kd_, int_pitch_i_limit_,
                                 -int_pitch_i_limit_);
  yaw_joint_.vel_pid_.setGains(20, 2, 0, 10, -10);
  pitch_joint_.vel_pid_.setGains(20, 2, 0, 10, -10);

  has_imu_ = controller_nh.hasParam("imu_name");
  if (has_imu_)
  {
    imu_name_ = controller_nh.param("imu_name", imu_name_, static_cast<std::string>("gimbal_imu"));

    imu_sub_ = controller_nh.subscribe("/gimbal_imu/data_filtered", 1, &HeroGimbalController::imuCallback, this);
    ROS_WARN_ONCE("has imu");
  }
  else
  {
    ROS_INFO("Param imu_name has not set, use motors' data instead of imu.");
  }

  ROS_INFO("[Hero_gimbal_controller] init v1.1 ");
  return true;
}

void HeroGimbalController::starting(const ros::Time& /*time*/)
{
}

void HeroGimbalController::update(const ros::Time& time, const ros::Duration& period)
{
  if ((ros::Time::now() - last_cmd_time_).toSec() > gimbal_cmd_timeout_)
  {
    gimbal_cmd_msg_ = rm_msgs_exam::GimbalCmd();
  }
  if (gimbal_cmd_msg_.mode == 0)
  {
    yaw_joint_.setVel(gimbal_cmd_msg_.rate_yaw);
    pitch_joint_.setVel(gimbal_cmd_msg_.rate_pitch);
  }
  if (gimbal_cmd_msg_.mode == 1)
  {
    yaw_joint_.setPos(gimbal_cmd_msg_.pos_yaw);
    pitch_joint_.setPos(gimbal_cmd_msg_.pos_pitch);
  }
  // Gravity Compensation
  yaw_joint_.setTorBias(0.0);
  pitch_joint_.setTorBias(0.0);
  if (enable_gravity_compensation_)
  {
    pitch_joint_.setTorBias(gravityFeedForward());
  }

  yaw_joint_.update(period);
  pitch_joint_.update(period);

  //debug
  static int debug_count = 0;
  debug_count++;
  if (debug_count % 10 == 0)
  {
    debug_pub_msgs.data.resize(8);
    debug_pub_msgs.data[0] = yaw_joint_.world_vel;
    debug_pub_msgs.data[1] = gimbal_cmd_msg_.rate_pitch;
    yaw_joint_.pos_pid_.getGains(debug_pub_msgs.data[2], debug_pub_msgs.data[5], debug_pub_msgs.data[3],
                                 debug_pub_msgs.data[5],
                                 debug_pub_msgs.data[5]);
    debug_pub_msgs.data[4] = test[0];
    debug_pub_msgs.data[6] = test[1];
    debug_pub_msgs.data[7] = pitch_joint_.joint_.getPosition();

    debug_pub_.publish(debug_pub_msgs);
  }
  //debug endf
}


double HeroGimbalController::gravityFeedForward()
{
  Eigen::Vector3d gravity(0, 0, -gravity_ * gimbal_mass_);
  Eigen::AngleAxisd pitch_rot(pitch_joint_.world_pos, Eigen::Vector3d::UnitY());
  Eigen::Vector3d gravity_rotated = pitch_rot * gravity;
  Eigen::Vector3d mass_origin(mass_origin_.x, 0, mass_origin_.z);
  test[0] = -mass_origin.cross(gravity_rotated).y();
  return -mass_origin.cross(gravity_rotated).y();
}

void HeroGimbalController::gimbalCmdCallback(const rm_msgs_exam::GimbalCmd::ConstPtr& msg)
{
  gimbal_cmd_msg_ = *msg;
  last_cmd_time_ = ros::Time::now();
}

void HeroGimbalController::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf2::Quaternion q(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w
      );
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw_joint_.world_pos = yaw;
  pitch_joint_.world_pos = -std::remainder(PI - roll, 2 * PI);
  yaw_joint_.world_vel = -msg->angular_velocity.z;
  pitch_joint_.world_vel = msg->angular_velocity.x;
}

void HeroGimbalController::debugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (msg->data.size() < 5)
  {
    ROS_WARN_THROTTLE(1.0, "[HeroGimbalController] debug msg too short: %zu", msg->data.size());
    return;
  }
  if (static_cast<int>(msg->data[0]) == 0) // cmd_state == 0 → PID 调试
  {
    int debug_id = static_cast<int>(msg->data[1]);
    switch (debug_id)
    {
      case 0:
        debug_pid = nullptr;
        return;
      case 1:
        debug_pid = &yaw_joint_.pos_pid_;
        break;
      case 2:
        debug_pid = &pitch_joint_.pos_pid_;
        break;
      case 3:
        debug_pid = &yaw_joint_.vel_pid_;
        break;
      case 4:
        debug_pid = &pitch_joint_.vel_pid_;
        break;
      default:
        // ROS_WARN_THROTTLE(1.0,
        //                   "[HeroGimbalController] unknown debug_id = %d (expect 1=yaw, 2=pitch)",
        //                   debug_id);
        return;
    }
    // setGains(p, i, d, i_max, i_min)
    debug_pid->setGains(msg->data[2], // Kp
                        msg->data[3], // Ki
                        msg->data[4], // Kd
                        10.0,         // i_max
                        -10.0);       // i_min
  }
  else
  {
  }
}

void HeroGimbalController::reconfigureCallback(hero_chassis_controller::GimbalControllerConfig& config, uint32_t level)
{
  // mass_origin_ = config.gimbal_mass;
}

PLUGINLIB_EXPORT_CLASS(hero_gimbal_controller::HeroGimbalController, controller_interface::ControllerBase)
}