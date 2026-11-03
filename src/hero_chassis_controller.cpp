//
// Created by qiayuan on 2/6/21.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid.h>
#include <Eigen/Dense>
#include <cmath>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace hero_chassis_controller
{
void Wheel_joint::setVel(double v)
{
  state = 2;
  vel_target_ = v;
}

void Wheel_joint::setTor(double tor)
{
  state = 1;
  tor_target_ = tor;
}

void Wheel_joint::setTorBias(double tor)
{
  tor_bias_ = tor;
}

void Wheel_joint::update(ros::Duration dt)
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
    double err = vel_target_ - joint_.getVelocity();
    joint_.setCommand(pid_.computeCommand(err, dt) + tor_bias_);
  }
}

double linear_max_acceleration = 3.0;
double angular_max_acceleration = 8.0;

void HeroChassisController::acceleration_limit(double wish_vel_x, double wish_vel_y, double wish_rad_z, double dt,
                                               double& cmd_vel_x_,
                                               double& cmd_vel_y_, double& cmd_rad_z_)
{
  if (dt <= 1e-5)
  {
    return;
  }
  // double temp_sin = sin(chassis_wz * dt);
  // double temp_cos = cos(chassis_wz * dt);
  // double dVx = wish_vel_x * temp_cos + wish_vel_y * temp_sin - cmd_vel_x_;
  // double dVy = wish_vel_x * -temp_sin + wish_vel_y * temp_cos - cmd_vel_y_;
  double dVx = wish_vel_x - cmd_vel_x_;
  double dVy = wish_vel_y - cmd_vel_y_;
  double dVLinear_sq = dVx * dVx + dVy * dVy;
  double accLinear_sq = linear_max_acceleration * linear_max_acceleration * dt * dt;
  if (dVLinear_sq > accLinear_sq)
  {
    double scale = std::sqrt(accLinear_sq / dVLinear_sq);
    dVx *= scale;
    dVy *= scale;
    cmd_vel_x_ += dVx;
    cmd_vel_y_ += dVy;
  }
  else
  {
    cmd_vel_x_ = wish_vel_x;
    cmd_vel_y_ = wish_vel_y;
  }

  double dWz = wish_rad_z - cmd_rad_z_;
  double dWz_max = angular_max_acceleration * dt;
  if (dWz > dWz_max)
  {
    cmd_rad_z_ += dWz_max;
  }
  else if (dWz < -dWz_max)
  {
    cmd_rad_z_ -= dWz_max;
  }
  else
  {
    cmd_rad_z_ = wish_rad_z;
  }
}

//scale_yaw: (wheel_track+wheel_base)/2
void forwardKinematics(double fl, double fr, double bl, double br, double scale_yaw, double& vx, double& vy,
                       double& rz)
{
  vx = (fl + fr + bl + br) * 0.25;
  vy = (bl - fl + fr - br) * 0.25;
  rz = (fr + br - fl - bl) * 0.25 / scale_yaw;
}


bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("wheel_track", wheel_track, 0.4);
  controller_nh.param("wheel_base", wheel_base, 0.4);
  controller_nh.param("max_effort", max_effort_, 15.0);
  controller_nh.param("global_vel_cmd", global_vel_cmd_en, false);
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
  controller_nh.param("pid_kp", init_kp_, 0.5);
  controller_nh.param("pid_ki", init_ki_, 0.0);
  controller_nh.param("pid_kd", init_kd_, 0.0);
  controller_nh.param("pid_i_limit", init_i_limit_, 20.0);
  controller_nh.param("odom_pub_en", odom_pub_en, false);
  controller_nh.param("linear_max_acceleration", linear_max_acceleration, 3.0);
  controller_nh.param("angular_max_acceleration", angular_max_acceleration, 8.0);

  front_left_joint_.joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_.joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_.joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_.joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
  front_left_joint_.pid_.setGains(init_kp_, init_ki_, init_kd_, init_i_limit_, -init_i_limit_);
  front_right_joint_.pid_.setGains(init_kp_, init_ki_, init_kd_, init_i_limit_, -init_i_limit_);
  back_left_joint_.pid_.setGains(init_kp_, init_ki_, init_kd_, init_i_limit_, -init_i_limit_);
  back_right_joint_.pid_.setGains(init_kp_, init_ki_, init_kd_, init_i_limit_, -init_i_limit_);

  cmd_vel_sub_ = controller_nh.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
  debug_sub_ = controller_nh.subscribe(
      "debug", 1, &HeroChassisController::debugCallback, this);

  debug_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("/chassis_controller/debug", 1);

  ROS_INFO("[HeroChassisController] init v1.2");
  if (odom_pub_en)
  {
    ROS_INFO("[HeroChassisController] odom_en");
    controller_nh.param<std::string>("odom_head_frame_id", odom_tf_.header.frame_id, "odom");
    controller_nh.param<std::string>("odom_child_frame_id", odom_tf_.child_frame_id, "base_link");

    controller_nh.param("init_x", chassis_x_, 0.0);
    controller_nh.param("init_y", chassis_y_, 0.0);
    controller_nh.param("init_yaw", chassis_yaw_, 0.0);
  }

  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/)
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>();
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  double dt = period.toSec();
  double scale_yaw = (wheel_base + wheel_track) / 2;
  //tf
  if (odom_pub_en)
  {
    forwardKinematics(front_left_joint_.joint_.getVelocity() * wheel_radius_,
                      front_right_joint_.joint_.getVelocity() * wheel_radius_,
                      back_left_joint_.joint_.getVelocity() * wheel_radius_,
                      back_right_joint_.joint_.getVelocity() * wheel_radius_,
                      scale_yaw, chassis_vx, chassis_vy, chassis_wz);
    double psi_mid = chassis_yaw_ + 0.5 * chassis_wz * dt; // 本周期中点朝向
    double c = std::cos(psi_mid), s = std::sin(psi_mid);
    chassis_x_ += (c * chassis_vx - s * chassis_vy) * dt;
    chassis_y_ += (s * chassis_vx + c * chassis_vy) * dt;
    chassis_yaw_ += chassis_wz * dt;

    publishTf(time);
  }
  //tf end
  if ((ros::Time::now() - last_cmd_time_).toSec() > cmd_vel_timeout_)
  {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;
  }
  acceleration_limit(cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z, dt,
                     cmd_vel_x, cmd_vel_y, cmd_rad_z);
  double chassis_cmd_vel_x = 0;
  double chassis_cmd_vel_y = 0;
  if (global_vel_cmd_en)
  {
    if (!odom_pub_en)
    {
      ROS_ERROR("[HeroChassisController] odom_pub_en is false,couldn't use global_vel_cmd_en");
      return;
    }
    double temp_sin = sin(chassis_yaw_);
    double temp_cos = cos(chassis_yaw_);
    chassis_cmd_vel_x = cmd_vel_x * temp_cos + cmd_vel_y * temp_sin;
    chassis_cmd_vel_y = cmd_vel_x * -temp_sin + cmd_vel_y * temp_cos;
  }
  else
  {
    chassis_cmd_vel_x = cmd_vel_x;
    chassis_cmd_vel_y = cmd_vel_y;
  }

  {
    double front_left_vel_target = chassis_cmd_vel_x - chassis_cmd_vel_y - cmd_rad_z * scale_yaw;
    double front_right_vel_target = chassis_cmd_vel_x + chassis_cmd_vel_y + cmd_rad_z * scale_yaw;
    double back_left_vel_target = chassis_cmd_vel_x + chassis_cmd_vel_y - cmd_rad_z * scale_yaw;
    double back_right_vel_target = chassis_cmd_vel_x - chassis_cmd_vel_y + cmd_rad_z * scale_yaw;
    front_left_joint_.setVel(front_left_vel_target / wheel_radius_);
    front_right_joint_.setVel(front_right_vel_target / wheel_radius_);
    back_left_joint_.setVel(back_left_vel_target / wheel_radius_);
    back_right_joint_.setVel(back_right_vel_target / wheel_radius_);
  }

  front_left_joint_.update(period);
  front_right_joint_.update(period);
  back_left_joint_.update(period);
  back_right_joint_.update(period);

  //debug
  static int debug_count = 0;
  debug_count++;
  if (debug_count % 10 == 0)
  {
    debug_pub_msgs.data.resize(3);
    debug_pub_msgs.data[0] = front_left_joint_.joint_.getVelocity();
    debug_pub_msgs.data[1] = cmd_vel_x;
    debug_pub_msgs.data[2] = chassis_vx;
    debug_pub_.publish(debug_pub_msgs);
  }
  //debug end
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_vel_ = *msg;
  last_cmd_time_ = ros::Time::now();
}


void HeroChassisController::debugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  double kp = msg->data[5];
  double ki = msg->data[6];
  double kd = msg->data[7];
  front_left_joint_.pid_.setGains(kp, ki, kd, 20, -20);
  front_right_joint_.pid_.setGains(kp, ki, kd, 20, -20);
  back_left_joint_.pid_.setGains(kp, ki, kd, 20, -20);
  back_right_joint_.pid_.setGains(kp, ki, kd, 20, -20);
  if (abs(msg->data[0] - 1) < 1e-3)
  {
    chassis_x_ = 0.0;
    chassis_y_ = 0.0;
    chassis_yaw_ = 0.0;
  }
}

void HeroChassisController::publishTf(const ros::Time& time)
{
  odom_tf_.header.stamp = time;
  if (!tf_broadcaster_)
    return;
  odom_tf_.transform.translation.x = chassis_x_;
  odom_tf_.transform.translation.y = chassis_y_;
  odom_tf_.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, chassis_yaw_);
  odom_tf_.transform.rotation.x = q.x();
  odom_tf_.transform.rotation.y = q.y();
  odom_tf_.transform.rotation.z = q.z();
  odom_tf_.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(odom_tf_);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}