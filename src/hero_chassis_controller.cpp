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
#include <rm_msgs_exam/ChassisCmd.h>

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
  controller_nh.param("pid_i_limit", init_i_limit_, 20.0);
  controller_nh.param("odom_pub_en", odom_pub_en, false);
  controller_nh.param("linear_max_acceleration", linear_max_acceleration, 3.0);
  controller_nh.param("angular_max_acceleration", angular_max_acceleration, 8.0);

  control_toolbox::Pid init_pid;
  ros::NodeHandle nh("wheel_pid_params");
  if (!init_pid.init(nh))
  {
    ROS_ERROR("[HeroChassisController]Failed to wheel PID from parameters.");
  }
  ros::NodeHandle nh_follow("follow_pid_params");
  if (!follow_pid_.init(nh))
  {
    ROS_WARN("[HeroChassisController]Failed to chassis follow PID from parameters.");
  }
  else
  {
    follow_pid_.setGains(4.0, 0, 0, 4, -4);
  }
  front_left_joint_.pid_ = init_pid;
  front_right_joint_.pid_ = init_pid;
  back_left_joint_.pid_ = init_pid;
  back_right_joint_.pid_ = init_pid;
  try
  {
    front_left_joint_.joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_joint_.joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_joint_.joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_joint_.joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR("[HeroGimbalController] getHandle failed: %s", e.what());
    return false;
  }
  cmd_vel_sub_ = controller_nh.subscribe<geometry_msgs::Twist>(
      "cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);
  chassis_cmd_sub_ = controller_nh.subscribe<rm_msgs_exam::ChassisCmd>(
      "cmd", 1, &HeroChassisController::chassisCmdCallback, this);
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
  front_left_joint_.pid_.reset();
  front_right_joint_.pid_.reset();
  back_left_joint_.pid_.reset();
  back_right_joint_.pid_.reset();
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // double dt = period.toSec();
  //tf///////
  if (odom_pub_en)
  {
    updateOdom(time, period);
  }

  cmd_vel_ = cmd_rt_buffer_.readFromRT()->cmd_vel_;
  chassis_cmd_ = cmd_rt_buffer_.readFromRT()->cmd_chassis_;
  if (state_ != chassis_cmd_.mode)
  {
    state_changed_ = true;
    state_ = chassis_cmd_.mode;
  }
  if (chassis_cmd_.accel.linear.x != 0 && chassis_cmd_.accel.angular.z != 0)
  {
    linear_max_acceleration = chassis_cmd_.accel.linear.x;
    angular_max_acceleration = chassis_cmd_.accel.angular.z;
  }

  if ((time - cmd_rt_buffer_.readFromRT()->stamp_).toSec() > cmd_vel_timeout_)
  {
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.linear.y = 0.0;
    cmd_vel_.angular.z = 0.0;
  }

  switch (state_)
  {
    case RAW:
      stateRaw();
      break;
    case FOLLOW:
      stateFollow(period);
      break;
    case TWIST:
      stateTwist();
      break;
    case WORLD:
      stateWorld();
      break;
    case SIN_SPIN:
      stateSinSpin(period);
      break;
    default:
      ROS_WARN("[hero_chassis_controller] state mismatch!");
      break;
  }
  chassis_acc_limit(cmd_vel_, chassis_cmd_vel_, chassis_yaw_, period);

  {
    double front_left_vel_target = chassis_cmd_vel_.linear.x - chassis_cmd_vel_.linear.y - chassis_cmd_vel_.angular.z *
                                   scale_yaw;
    double front_right_vel_target = chassis_cmd_vel_.linear.x + chassis_cmd_vel_.linear.y + chassis_cmd_vel_.angular.z *
                                    scale_yaw;
    double back_left_vel_target = chassis_cmd_vel_.linear.x + chassis_cmd_vel_.linear.y - chassis_cmd_vel_.angular.z *
                                  scale_yaw;
    double back_right_vel_target = chassis_cmd_vel_.linear.x - chassis_cmd_vel_.linear.y + chassis_cmd_vel_.angular.z *
                                   scale_yaw;
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
    debug_pub_msgs.data[1] = chassis_cmd_vel_.linear.x;
    debug_pub_msgs.data[2] = chassis_vx_;
    debug_pub_.publish(debug_pub_msgs);
  }
  //debug end
}

void HeroChassisController::stateRaw()
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter RAW");
  }
}

void HeroChassisController::stateWorld()
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter WORLD");
  }
  rotateCmdVel(cmd_vel_, cmd_vel_, chassis_yaw_);
}

void HeroChassisController::stateFollow(ros::Duration period)
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter FOLLOW");
  }
  rotateCmdVel(cmd_vel_, cmd_vel_, rale_gimbal_yaw_pos);
  cmd_vel_.angular.z = follow_pid_.computeCommand(rale_gimbal_yaw_pos, period) + chassis_cmd_.follow_vel_des;
}

void HeroChassisController::stateTwist()
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter TWIST");
  }
  rotateCmdVel(cmd_vel_, cmd_vel_, rale_gimbal_yaw_pos);
}

void HeroChassisController::stateSinSpin(ros::Duration period)
{
  if (state_changed_)
  {
    state_changed_ = false;
    ROS_INFO("[Chassis] Enter SIN_SPIN");
  }
  rotateCmdVel(cmd_vel_, cmd_vel_, rale_gimbal_yaw_pos);
  static double step = 0;
  static double sin_period = 2;
  step += 2 * PI * period.toSec() / sin_period;
  if (step > 2 * PI)
  {
    step -= 2 * PI;
  }
  cmd_vel_.angular.z = 2 * sin(step) + 7;
}


void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  cmd_struct_.cmd_vel_ = *msg;
  cmd_struct_.stamp_ = ros::Time::now();
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}

void HeroChassisController::chassisCmdCallback(const rm_msgs_exam::ChassisCmd::ConstPtr& msg)
{
  cmd_struct_.cmd_chassis_ = *msg;
  cmd_rt_buffer_.writeFromNonRT(cmd_struct_);
}


control_toolbox::Pid* debug_pid = nullptr;

void HeroChassisController::debugCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if (msg->data.size() < 8)
  {
    ROS_ERROR_THROTTLE(1.0, "[Chassis] Debug msg data size is too small!");
    return;
  }
  if (static_cast<int>(msg->data[7]) == 1)
  {
    reset();
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
        debug_pid = &follow_pid_;
        break;
      case 2:
        debug_pid = &front_left_joint_.pid_;
        break;
      default:
        debug_pid = nullptr;
        return;
    }
    // setGains(p, i, d, i_max, i_min)
    debug_pid->setGains(msg->data[2], // Kp
                        msg->data[3], // Ki
                        msg->data[4], // Kd
                        10.0,         // i_max
                        -10.0);       // i_min
  }
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
                    scale_yaw, chassis_vx_, chassis_vy_, chassis_wz_);
  double psi_mid = chassis_yaw_ + 0.5 * chassis_wz_ * dt; // 本周期中点朝向
  double c = std::cos(psi_mid), s = std::sin(psi_mid);
  chassis_x_ += (c * chassis_vx_ - s * chassis_vy_) * dt;
  chassis_y_ += (s * chassis_vx_ + c * chassis_vy_) * dt;
  chassis_yaw_ += chassis_wz_ * dt;

  publishTf(time);
}

void HeroChassisController::reset()
{
  chassis_x_ = 0.0;
  chassis_y_ = 0.0;
  chassis_yaw_ = 0.0;
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

void HeroChassisController::rotateCmdVel(const geometry_msgs::Twist& origin, geometry_msgs::Twist& output, double yaw)
{
  Eigen::Vector2d v(origin.linear.x, origin.linear.y);
  Eigen::Rotation2Dd rotation(yaw);
  Eigen::Matrix2d rot_mat = rotation.toRotationMatrix();
  Eigen::Vector2d v_rotated = rot_mat * v;
  output.linear.x = v_rotated(0);
  output.linear.y = v_rotated(1);
}

void HeroChassisController::chassisAccLimit(const geometry_msgs::Twist& origin, geometry_msgs::Twist& output,
                                            double yaw,
                                            ros::Duration period)
{
  static double last_yaw = yaw;
  double dt = period.toSec();
  rotateCmdVel(output, output, last_yaw - yaw);
  last_yaw = yaw;
  double dVx = origin.linear.x - output.linear.x;
  double dVy = origin.linear.y - output.linear.y;
  double dVLinear_sq = dVx * dVx + dVy * dVy;
  double accLinear_sq = linear_max_acceleration * linear_max_acceleration * dt * dt;
  if (dVLinear_sq > accLinear_sq)
  {
    double scale = std::sqrt(accLinear_sq / dVLinear_sq);
    dVx *= scale;
    dVy *= scale;
    output.linear.x += dVx;
    output.linear.y += dVy;
  }
  else
  {
    output.linear.x = origin.linear.x;
    output.linear.y = origin.linear.y;
  }

  double dWz = origin.angular.z - output.angular.z;
  double dWz_max = angular_max_acceleration * dt;
  if (dWz > dWz_max)
  {
    output.angular.z += dWz_max;
  }
  else if (dWz < -dWz_max)
  {
    output.angular.z -= dWz_max;
  }
  else
  {
    output.angular.z = origin.angular.z;
  }
}


PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}