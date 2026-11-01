//
// Created by qiayuan on 2/6/21.
//

#include "simple_chassis_controller/simple_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <control_toolbox/pid.h>

namespace simple_chassis_controller {
    void Wheel_joint::setVel(double v) {
        state = 2;
        vel_target_ = v;
    }

    void Wheel_joint::setTor(double tor) {
        state = 1;
        tor_target_ = tor;
    }

    void Wheel_joint::update(ros::Duration dt) {
        if (state == 0) {
            joint_.setCommand(0);
        } else if (state == 1) {
            joint_.setCommand(tor_target_);
        } else if (state == 2) {
            double err = vel_target_ - joint_.getVelocity();
            joint_.setCommand(pid_.computeCommand(err, dt));
        }
    }

    constexpr double linear_max_acceleration = 3.0;
    constexpr double angular_max_acceleration = 8.0;

    void acceleration_limit(double wish_vel_x, double wish_vel_y, double wish_rad_z, double dt, double &cmd_vel_x,
                            double &cmd_vel_y, double &cmd_rad_z) {
        if (dt <= 1e-5) {
            return;
        }
        double dVx = wish_vel_x - cmd_vel_x;
        double dVy = wish_vel_y - cmd_vel_y;
        double dVLinear_sq = dVx * dVx + dVy * dVy;
        double accLinear_sq = linear_max_acceleration * linear_max_acceleration * dt * dt;
        if (dVLinear_sq > accLinear_sq) {
            double scale = std::sqrt(accLinear_sq / dVLinear_sq);
            dVx *= scale;
            dVy *= scale;
            cmd_vel_x += dVx;
            cmd_vel_y += dVy;
        } else {
            cmd_vel_x = wish_vel_x;
            cmd_vel_y = wish_vel_y;
        }

        double dWz = wish_rad_z - cmd_rad_z;
        double dWz_max = angular_max_acceleration * dt;
        if (dWz > dWz_max) {
            cmd_rad_z += dWz_max;
        } else if (dWz < -dWz_max) {
            cmd_rad_z -= dWz_max;
        } else {
            cmd_rad_z = wish_rad_z;
        }
    }

    //scale_yaw: (wheel_track+wheel_base)/2
    void forwardKinematics(double fl, double fr, double bl, double br, double scale_yaw, double &vx, double &vy,
                           double &rz) {
        vx = (fl + fr + bl + br) * 0.25;
        vy = (bl - fl + fr - br) * 0.25;
        rz = (fr + br - fl + bl) * 4 / scale_yaw;
    }


    bool SimpleChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                       ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
        controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
        controller_nh.param("wheel_track", wheel_track, 0.4);
        controller_nh.param("wheel_base", wheel_base, 0.4);
        controller_nh.param("max_effort", max_effort_, 15.0);
        controller_nh.param("force_col", chassis_vel_col, false);
        controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
        controller_nh.param("pid_kp", init_kp_, 1.0);
        controller_nh.param("pid_ki", init_ki_, 0.01);
        controller_nh.param("pid_kd", init_kd_, 0.0);
        controller_nh.param("pid_i_limit", init_i_limit_, 20.0);
        controller_nh.param("odom_pub_en", odom_pub_en, false);

        front_left_joint_.joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
        front_right_joint_.joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
        back_left_joint_.joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
        back_right_joint_.joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");
        front_left_joint_.pid_.setGains(init_kp_, init_ki_, init_kd_, init_i_limit_, -init_i_limit_);
        front_right_joint_.pid_.setGains(init_kp_, init_ki_, init_kd_, init_i_limit_, -init_i_limit_);
        back_left_joint_.pid_.setGains(init_kp_, init_ki_, init_kd_, init_i_limit_, -init_i_limit_);
        back_right_joint_.pid_.setGains(init_kp_, init_ki_, init_kd_, init_i_limit_, -init_i_limit_);


        cmd_vel_sub_ = controller_nh.subscribe<geometry_msgs::Twist>(
            "cmd_vel", 1, &SimpleChassisController::cmdVelCallback, this);
        debug_sub_ = controller_nh.subscribe(
            "debug", 1, &SimpleChassisController::debugCallback, this);

        debug_pub_ = controller_nh.advertise<std_msgs::Float64MultiArray>("/chassis_controller/debug", 1);

        ROS_INFO("[SimpleChassisController] init v1.1");
        if (odom_pub_en) {
            controller_nh.param<std::string>("odom_head_frame_id", odom_tf_.header.frame_id, "odom");
            controller_nh.param<std::string>("odom_child_frame_id", odom_tf_.child_frame_id, "base_link");

            controller_nh.param("init_x", chassis_x_, 0.0);
            controller_nh.param("init_y", chassis_y_, 0.0);
            controller_nh.param("init_yaw", chassis_yaw_, 0.0);
        }

        return true;
    }

    void SimpleChassisController::starting(const ros::Time & /*time*/) {
    }

    void SimpleChassisController::update(const ros::Time &time, const ros::Duration &period) {
        double dt = period.toSec();
        double scale_yaw = (wheel_base + wheel_track) / 2;
        //tf
        if (odom_pub_en) {
            forwardKinematics(front_left_joint_.joint_.getVelocity() * wheel_radius_,
                              front_right_joint_.joint_.getVelocity() * wheel_radius_,
                              back_left_joint_.joint_.getVelocity() * wheel_radius_,
                              back_right_joint_.joint_.getVelocity() * wheel_radius_,
                              scale_yaw, chassis_vx, chassis_vy, chassis_wz);
        }


        publishTf(time);
        //tf end
        acceleration_limit(cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z, dt,
                           cmd_vel_x, cmd_vel_y, cmd_rad_z);
        if ((ros::Time::now() - last_cmd_time_).toSec() > cmd_vel_timeout_) {
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.linear.y = 0.0;
            cmd_vel_.angular.z = 0.0;
        }
        if (chassis_vel_col) {
            // front_left_vel_target = 0;
        } else {
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
        if (debug_count % 10 == 0) {
            debug_pub_msgs.data.resize(3);
            debug_pub_msgs.data[0] = front_left_joint_.joint_.getVelocity();
            debug_pub_msgs.data[1] = front_left_joint_.getVel_target();
            debug_pub_msgs.data[2] = front_left_joint_.joint_.getEffort();
            debug_pub_.publish(debug_pub_msgs);
        }
        //debug end
    }

    void SimpleChassisController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
        cmd_vel_ = *msg;
        last_cmd_time_ = ros::Time::now();
    }

    void SimpleChassisController::debugCallback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
        double kp = msg->data[5];
        double ki = msg->data[6];
        double kd = msg->data[7];
        front_left_joint_.pid_.setGains(kp, ki, kd, 20, -20);
        front_right_joint_.pid_.setGains(kp, ki, kd, 20, -20);
        back_left_joint_.pid_.setGains(kp, ki, kd, 20, -20);
        back_right_joint_.pid_.setGains(kp, ki, kd, 20, -20);
    }

    void SimpleChassisController::publishTf(const ros::Time &time) {
        odom_tf_.header.stamp = time;

        odom_tf_.transform.translation.x = chassis_x_;
        odom_tf_.transform.translation.y = chassis_y_;
        odom_tf_.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, chassis_yaw_);
        odom_tf_.transform.rotation.x = q.x();
        odom_tf_.transform.rotation.y = q.y();
        odom_tf_.transform.rotation.z = q.z();
        odom_tf_.transform.rotation.w = q.w();

        tf_broadcaster_.sendTransform(odom_tf_);
    }

    PLUGINLIB_EXPORT_CLASS(simple_chassis_controller::SimpleChassisController, controller_interface::ControllerBase)
}
