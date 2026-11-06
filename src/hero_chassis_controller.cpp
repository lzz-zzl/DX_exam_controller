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

  control_toolbox::Pid init_pid;
  ros::NodeHandle nh("wheel_pid_params");
  if (!init_pid.init(nh))
  {
    ROS_ERROR("Failed to initialize PID from parameters.");
  }
  front_left_joint_.pid_ = init_pid;
  front_right_joint_.pid_ = init_pid;
  back_left_joint_.pid_ = init_pid;
  back_right_joint_.pid_ = init_pid;
  front_left_joint_.joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_.joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_.joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_.joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  cmd_vel_sub_ = controller_nh.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
  debug_sub_ = controller_nh.subscribe(
      "debug", 1, &HeroChassisController::debugCallback, this);
  joint_state_sub_ = controller_nh.subscribe("/joint_states", 1, &HeroChassisController::jointStatesCallback, this);
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
  //tf
  if (odom_pub_en)
  {
    updateOdom(time, period);
  }

  cmd_vel_ = cmd_rt_buffer_.readFromRT()->cmd_vel_;

  if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > cmd_vel_timeout_)
  {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;
  }

  switch (state_)
  {
    case RAW:
      state_raw();
      break;
    case WORLD:
      state_world();
      break;
    case FOLLOW:
      state_follow();
      break;
    case TWIST:
      state_twist();
      break;
    default:
      ROS_ERROR("[hero_chassis_controller] state mismatch!");
      break;
  }

  acceleration_limit(cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z, dt,
                     cmd_vel_x, cmd_vel_y, cmd_rad_z);
  // cmd_vel_x = cmd_vel_.linear.x;
  // cmd_vel_y = cmd_vel_.linear.y;
  // cmd_rad_z = cmd_vel_.angular.z;
  {
    double front_left_vel_target = cmd_vel_x - cmd_vel_y - cmd_rad_z * scale_yaw;
    double front_right_vel_target = cmd_vel_x + cmd_vel_y + cmd_rad_z * scale_yaw;
    double back_left_vel_target = cmd_vel_x + cmd_vel_y - cmd_rad_z * scale_yaw;
    double back_right_vel_target = cmd_vel_x - cmd_vel_y + cmd_rad_z * scale_yaw;
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

void HeroChassisController::state_raw()
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter RAW");
  }
}

void HeroChassisController::state_world()
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter WORLD");
  }
  double temp_sin = sin(chassis_yaw_);
  double temp_cos = cos(chassis_yaw_);
  double temp_vel_x = cmd_vel_.linear.x * temp_cos + cmd_vel_.linear.y * temp_sin;
  double temp_vel_y = cmd_vel_.linear.x * -temp_sin + cmd_vel_.linear.y * temp_cos;
  cmd_vel_.linear.x = temp_vel_x;
  cmd_vel_.linear.y = temp_vel_y;
}

void HeroChassisController::state_follow()
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter FOLLOW");
  }
}

void HeroChassisController::state_twist()
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter TWISE");
  }
  double temp_sin = sin(-rale_gimbal_yaw_pos);
  double temp_cos = cos(-rale_gimbal_yaw_pos);
  double temp_vel_x = cmd_vel_.linear.x * temp_cos + cmd_vel_.linear.y * temp_sin;
  double temp_vel_y = cmd_vel_.linear.x * -temp_sin + cmd_vel_.linear.y * temp_cos;
  cmd_vel_.linear.x = temp_vel_x;
  cmd_vel_.linear.y = temp_vel_y;
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_struct_.cmd_vel_ = *msg;
  cmd_struct_.stamp_ = ros::Time::now();
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}


void HeroChassisController::debugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (msg->data.size() < 8)
  {
    ROS_ERROR_THROTTLE(1.0, "[Chassis] Debug msg data size is too small!");
    return;
  }
  if (static_cast<int>(msg->data[5]) != state_)
  {
    state_changed_ = true;
    state_ = static_cast<int>(msg->data[5]);
  }
  // double kp = msg->data[5];
  // double ki = msg->data[6];
  // double kd = msg->data[7];

  // front_left_joint_.pid_.setGains(kp, ki, kd, 20, -20);
  // front_right_joint_.pid_.setGains(kp, ki, kd, 20, -20);
  // back_left_joint_.pid_.setGains(kp, ki, kd, 20, -20);
  // back_right_joint_.pid_.setGains(kp, ki, kd, 20, -20);
  // if (abs(msg->data[0] - 1) < 1e-3)
  // {
  //   chassis_x_ = 0.0;
  //   chassis_y_ = 0.0;
  //   chassis_yaw_ = 0.0;
  // }
}

void HeroChassisController::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (msg->name.size() != msg->position.size())
  {
    ROS_WARN_THROTTLE(1.0, "[Chassis] JointState name and position size mismatch!");
    return;
  }
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i] == "yaw_joint")
    {
      rale_gimbal_yaw_pos = std::remainder(msg->position[i], 2 * PI);
    }
  }
}


void HeroChassisController::updateOdom(const ros::Time& time, const ros::Duration& period)
{
  double dt = period.toSec();
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