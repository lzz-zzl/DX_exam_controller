//
// Created by root on 2026/9/21.
//

#ifndef SRC_PID_H
#define SRC_PID_H


#ifndef PID_H
#define PID_H

class PID
{
public:
  PID(double kp = 0, double ki = 0, double kd = 0,
      double out_min = -10, double out_max = 10, double der_alpha = 0.04,
      double integral_min = -1e9, double integral_max = 1e9)
    : kp_(kp)
    , ki_(ki)
    , kd_(kd)
    , out_min_(out_min)
    , out_max_(out_max)
    , der_alpha_(der_alpha)
    , integral_min_(integral_min)
    , integral_max_(integral_max)
    , prev_error_(0.0)
    , integral_(0.0)
    , first_run_(true)
  {
  }

  // 设置目标值
  void setTarget(double target) { target_ = target; }
  double getTarget() { return target_; }
  // 设置增益
  void setGains(double kp, double ki, double kd, double i_max, double i_min)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  // 重置
  void reset()
  {
    prev_error_ = 0.0;
    integral_ = 0.0;
    first_run_ = true;
  }

  // 计算输出
  // current: 当前值
  // dt: 时间步长（秒）
  double compute(double current, double dt)
  {
    if (dt <= 0.0)
      return 0.0;

    double error = target_ - current;

    // 积分项（带限幅，防积分饱和）
    integral_ += error * dt;
    if (integral_ > integral_max_)
      integral_ = integral_max_;
    if (integral_ < integral_min_)
      integral_ = integral_min_;

    // 微分项（首帧不计算，避免微分冲击）
    double derivative = 0.0;
    if (first_run_)
    {
      first_run_ = false;
    }
    else
    {
      derivative = (error - prev_error_) / dt;
      derivative = derivative * der_alpha_ + prev_der_ * (1 - der_alpha_);
    }
    prev_der_ = derivative;
    prev_error_ = error;

    // 输出
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // 输出限幅
    if (output > out_max_)
      output = out_max_;
    if (output < out_min_)
      output = out_min_;

    return output;
  }

  double computeCommand(double error, ros::Duration period)
  {
    double dt = period.toSec();
    if (dt <= 0.0)
      return 0.0;

    // 积分项（带限幅，防积分饱和）
    integral_ += error * dt;
    if (integral_ > integral_max_)
      integral_ = integral_max_;
    if (integral_ < integral_min_)
      integral_ = integral_min_;

    // 微分项（首帧不计算，避免微分冲击）
    double derivative = 0.0;
    if (first_run_)
    {
      first_run_ = false;
    }
    else
    {
      derivative = (error - prev_error_) / dt;
      derivative = derivative * der_alpha_ + prev_der_ * (1 - der_alpha_);
    }

    prev_der_ = derivative;
    prev_error_ = error;

    // 输出
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // 输出限幅
    if (output > out_max_)
      output = out_max_;
    if (output < out_min_)
      output = out_min_;

    return output;
  }


  void getGains(double& kp, double& ki, double& kd, double& i_max, double& i_min)
  {
    kp = kp_;
    ki = ki_;
    kd = kd_;
    i_max = integral_max_;
    i_min = integral_min_;
  }

private:
  double kp_, ki_, kd_;
  double out_min_, out_max_;
  double der_alpha_;
  double integral_min_, integral_max_;
  double target_ = 0.0;
  double prev_error_;
  double prev_der_;
  double integral_;
  bool first_run_;
};

#endif // PID_H

#endif //SRC_PID_H
