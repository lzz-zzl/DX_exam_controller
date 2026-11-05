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
  double dt = period.toSec();
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
    double err = vel_target_ - joint_.getVelocity();
    joint_.setCommand(vel_pid_.computeCommand(err, dt) + tor_bias_);
  }
  else if (state == 3)
  {
    double err = pos_target_ - world_pos;
    err = std::remainder(err, 2 * PI);
    test[0] = err;
    test[1] = pos_pid_.computeCommand(err, dt) + tor_bias_;
    joint_.setCommand(test[1]);
    // joint_.setCommand(pos_pid_.computeCommand(err, dt) + tor_bias_);
  }
}

PID* debug_pid = nullptr;

bool HeroGimbalController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  controller_nh.param("gimbal_cmd_timeout", gimbal_cmd_timeout_, 0.5);
  double init_yaw_kp_{}, init_yaw_ki_{}, init_yaw_kd_{}, int_yaw_i_limit_{},
         init_pitch_kp_{}, init_pitch_ki_{}, init_pitch_kd_{}, int_pitch_i_limit_{};
  controller_nh.param("pid_yaw_kp", init_yaw_kp_, 7.0);
  controller_nh.param("pid_yaw_ki", init_yaw_ki_, 0.0);
  controller_nh.param("pid_yaw_kd", init_yaw_kd_, 1.4);
  controller_nh.param("int_yaw_i_limit", int_yaw_i_limit_, 2.0);
  controller_nh.param("pid_pitch_kp", init_pitch_kp_, 8.0);
  controller_nh.param("pid_pitch_ki", init_pitch_ki_, 0.2);
  controller_nh.param("pid_pitch_kd", init_pitch_kd_, 1.0);
  controller_nh.param("int_pitch_i_limit", int_pitch_i_limit_, 2.0);
  debug_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("/hero_gimbal_controller/pub_debug", 1);
  gimbal_cmd_sub_ = controller_nh.subscribe("/controller/gimbal_controller/cmd", 1,
                                            &HeroGimbalController::gimbalCmdCallback,
                                            this);
  imu_sub_ = controller_nh.subscribe("/gimbal_imu/data_filtered", 1, &HeroGimbalController::imuCallback, this);
  debug_sub_ = controller_nh.subscribe("/hero_gimbal_controller/sub_debug", 1, &HeroGimbalController::debugCallback,
                                       this);
  try
  {
    pitch_joint.joint_ = effort_joint_interface->getHandle("pitch_joint");
    yaw_joint.joint_ = effort_joint_interface->getHandle("yaw_joint");
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR("[HeroGimbalController] getHandle failed: %s", e.what());
    return false;
  }
  yaw_joint.pos_pid_.setGains(init_yaw_kp_, init_yaw_ki_, init_yaw_kd_, int_yaw_i_limit_, -int_yaw_i_limit_);
  pitch_joint.pos_pid_.setGains(init_pitch_kp_, init_pitch_ki_, init_pitch_kd_, int_pitch_i_limit_,
                                -int_pitch_i_limit_);

  ROS_INFO("[Hero_gimbal_controller] init v1.1 ");
  return true;
}

void HeroGimbalController::starting(const ros::Time& /*time*/)
{
}

void HeroGimbalController::update(const ros::Time& time, const ros::Duration& period)
{
  // double dt = period.toSec();
  if ((ros::Time::now() - last_cmd_time_).toSec() > gimbal_cmd_timeout_)
  {
    gimbal_cmd_msg_ = rm_msgs_exam::GimbalCmd();
  }
  if (gimbal_cmd_msg_.mode == 0)
  {
    // yaw_joint.getTor_target();
  }
  if (gimbal_cmd_msg_.mode == 1)
  {
    yaw_joint.setPos(gimbal_cmd_msg_.pos_yaw);
    pitch_joint.setPos(gimbal_cmd_msg_.pos_pitch);
  }
  yaw_joint.update(period);
  pitch_joint.update(period);

  //debug
  static int debug_count = 0;
  debug_count++;
  if (debug_count % 10 == 0)
  {
    debug_pub_msgs.data.resize(8);
    debug_pub_msgs.data[0] = pitch_joint.world_pos;
    debug_pub_msgs.data[1] = gimbal_cmd_msg_.pos_pitch;
    yaw_joint.pos_pid_.getGains(debug_pub_msgs.data[2], debug_pub_msgs.data[5], debug_pub_msgs.data[3],
                                debug_pub_msgs.data[5],
                                debug_pub_msgs.data[5]);
    debug_pub_msgs.data[4] = test[0];
    debug_pub_msgs.data[6] = test[1];
    debug_pub_msgs.data[7] = pitch_joint.joint_.getPosition();

    debug_pub_.publish(debug_pub_msgs);
  }
  //debug endf
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
  yaw_joint.world_pos = yaw;
  pitch_joint.world_pos = -std::remainder(PI - roll, 2 * PI);
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
        break;
      case 1:
        debug_pid = &yaw_joint.pos_pid_;
        break;
      case 2:
        debug_pid = &pitch_joint.pos_pid_;
        break;
      case 3:
        debug_pid = &yaw_joint.vel_pid_;
        break;
      case 4:
        debug_pid = &pitch_joint.vel_pid_;
        break;
      default:
        ROS_WARN_THROTTLE(1.0,
                          "[HeroGimbalController] unknown debug_id = %d (expect 1=yaw, 2=pitch)",
                          debug_id);
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

PLUGINLIB_EXPORT_CLASS(hero_gimbal_controller::HeroGimbalController, controller_interface::ControllerBase)
}