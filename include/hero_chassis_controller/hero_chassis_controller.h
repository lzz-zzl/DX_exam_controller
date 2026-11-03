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

namespace hero_chassis_controller {
  class Wheel_joint {
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

  class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
  public:
    HeroChassisController() = default;

    ~HeroChassisController() override = default;

    static void acceleration_limit(double wish_vel_x, double wish_vel_y, double wish_rad_z, double dt,
                                   double &cmd_vel_x,
                                   double &cmd_vel_y, double &cmd_rad_z);

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
    ros::Time last_cmd_time_;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);

    ros::Subscriber debug_sub_;

    void debugCallback(const std_msgs::Float64MultiArray::ConstPtr &msg);

    ros::Publisher debug_pub_;
    std_msgs::Float64MultiArray debug_pub_msgs;

    double cmd_vel_x = 0, cmd_vel_y = 0, cmd_rad_z = 0; //限制处理速度
    geometry_msgs::Twist cmd_vel_; //初始命令速度

    //地盘
    double wheel_base = 0.4;
    double wheel_track = 0.4;
    double chassis_vx = 0, chassis_vy = 0, chassis_wz = 0;

    bool global_vel_cmd_en = false;
    bool odom_pub_en = false;

    double max_effort_ = 100;
    double wheel_radius_ = 0.07625;
    double cmd_vel_timeout_ = 0.5; // cmd_vel 超时（秒）
    double max_wheel_velocity_ = 100.0; // 轮子最大角速度 (rad/s)


    //tf
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::TransformStamped odom_tf_;
    double chassis_x_ = 0.0;
    double chassis_y_ = 0.0;
    double chassis_yaw_ = 0.0;

    void publishTf(const ros::Time &time);
  };
} // namespace hero_chassis_controller

#endif //HERO_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
