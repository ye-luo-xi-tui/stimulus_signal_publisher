//
// Created by yezi on 24-1-31.
//

#pragma once

#include "stimulus_signal_publisher/SetPublisherStatus.h"
#include <ros/time.h>
#include <XmlRpcValue.h>

namespace stimulus_signal_publisher
{
class SignalProducerBase
{
public:
  explicit SignalProducerBase(ros::NodeHandle& nh)
  {
    service_server_ = nh.advertiseService("set_publisher_status", &SignalProducerBase::setPublisherStatus, this);
    ROS_INFO("Ready to set status.");
  }
  virtual ~SignalProducerBase() = default;
  virtual double output() = 0;

protected:
  bool setPublisherStatus(stimulus_signal_publisher::SetPublisherStatus::Request& req,
                          stimulus_signal_publisher::SetPublisherStatus::Response& res)
  {
    if (status_ == stimulus_signal_publisher::SetPublisherStatus::Request::RESET &&
        req.status == stimulus_signal_publisher::SetPublisherStatus::Request::START)
      start_time_ = ros::Time::now();
    status_ = req.status;
    res.is_success = true;
    ROS_INFO("Switch to status: %d", status_);
    return true;
  }
  ros::ServiceServer service_server_;
  ros::Time start_time_;
  int status_ = stimulus_signal_publisher::SetPublisherStatus::Request::RESET;
};

class SinWave : public SignalProducerBase
{
public:
  SinWave(double magnitude, XmlRpc::XmlRpcValue frequency_points_config, ros::NodeHandle& nh) : SignalProducerBase(nh)
  {
    magnitude_ = magnitude;
    ROS_ASSERT(frequency_points_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
    frequency_points_.reserve(frequency_points_config.size());
    for (int i = 0; i < frequency_points_config.size(); i++)
      frequency_points_.push_back(frequency_points_config[i]);
    current_point_it_ = frequency_points_.begin();
  }

  double output() override
  {
    double output;
    if (status_ == stimulus_signal_publisher::SetPublisherStatusRequest::START)
    {
      if ((ros::Time::now() - start_time_).toSec() >= 20 * 1 / *current_point_it_)
      {
        if ((current_point_it_ + 1) == frequency_points_.end())
        {
          status_ = stimulus_signal_publisher::SetPublisherStatusRequest::RESET;
          current_point_it_ = frequency_points_.begin();
        }
        else
        {
          start_time_ = start_time_ + ros::Duration(20 * 1 / *current_point_it_);
          current_point_it_++;
        }
      }
      output = magnitude_ * std::sin(2 * M_PI * *current_point_it_ * (ros::Time::now() - start_time_).toSec());
    }
    if (status_ == stimulus_signal_publisher::SetPublisherStatusRequest::RESET)
      output = 0.;
    return output;
  }

private:
  double magnitude_;
  std::vector<double> frequency_points_;
  std::vector<double>::iterator current_point_it_;
};

class ConstantVelocity : public SignalProducerBase
{
public:
  ConstantVelocity(double start_point, double end_point, XmlRpc::XmlRpcValue velocity_config, ros::NodeHandle& nh)
    : SignalProducerBase(nh)
  {
    start_point_ = start_point;
    end_point_ = end_point;
    ROS_ASSERT(velocity_config.getType() == XmlRpc::XmlRpcValue::TypeArray);
    velocity_.reserve(velocity_config.size());
    for (int i = 0; i < velocity_config.size(); i++)
      velocity_.push_back(velocity_config[i]);
    current_point_it_ = velocity_.begin();
  }
  double output() override
  {
    double output;
    double switch_point = (end_point_ - start_point_) / *current_point_it_;
    if (status_ == stimulus_signal_publisher::SetPublisherStatusRequest::START)
    {
      double dt = (ros::Time::now() - start_time_).toSec();
      if (dt >= 2 * switch_point)
      {
        if ((current_point_it_ + 1) == velocity_.end())
        {
          status_ = stimulus_signal_publisher::SetPublisherStatusRequest::RESET;
          current_point_it_ = velocity_.begin();
        }
        else
        {
          start_time_ = start_time_ + ros::Duration(2 * (end_point_ - start_point_) / *current_point_it_);
          dt = 0.;
          current_point_it_++;
        }
      }
      if (dt <= switch_point)
        output = start_point_ + *current_point_it_ * dt;
      else
        output = end_point_ - *current_point_it_ * (dt - switch_point);
    }
    if (status_ == stimulus_signal_publisher::SetPublisherStatusRequest::RESET)
      output = start_point_;
    return output;
  }

private:
  double start_point_;
  double end_point_;
  std::vector<double> velocity_;
  std::vector<double>::iterator current_point_it_;
};
}  // namespace stimulus_signal_publisher
