// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROL_FILTERS__LOW_PASS_FILTER_HPP_
#define CONTROL_FILTERS__LOW_PASS_FILTER_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "control_toolbox/parameter_handler.hpp"
#include "filters/filter_base.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace control_filters
{
class LowPassParameters : public control_toolbox::ParameterHandler
{
public:
  explicit LowPassParameters(const std::string & params_prefix)
  : control_toolbox::ParameterHandler(params_prefix, 0, 1, 3)
  {
    add_double_parameter("sampling_frequency", false);
    add_double_parameter("damping_frequency", false);
    add_double_parameter("damping_intensity", false);

    add_integer_parameter("divider", false);
  }

  bool check_if_parameters_are_valid() override
  {
    bool ret = true;

    // Check if any string parameter is empty
    ret = !empty_parameter_in_list(string_parameters_);

    for (size_t i = 0; i < 3; ++i)
    {
      if (std::isnan(double_parameters_[i].second))
      {
        RCUTILS_LOG_ERROR_NAMED(
          logger_name_.c_str(), "Parameter '%s' has to be set",
          double_parameters_[i].first.name.c_str());
        ret = false;
      }
    }

    if (integer_parameters_[0].second < 0)
    {
      RCUTILS_LOG_ERROR_NAMED(
        logger_name_.c_str(), "Parameter '%s' has to be positive",
        integer_parameters_[0].first.name.c_str());
    }

    return ret;
  }

  void update_storage() override
  {
    sampling_frequency_ = double_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Sampling frequency is %e", sampling_frequency_);
    damping_frequency_ = double_parameters_[1].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Damping frequency is %e", damping_frequency_);
    damping_intensity_ = double_parameters_[2].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Damping intensity is %e", damping_intensity_);

    divider_ = integer_parameters_[0].second;
    RCUTILS_LOG_INFO_NAMED(logger_name_.c_str(), "Divider %d", divider_);

    a1_ = exp(
      -1.0 / sampling_frequency_ * (2.0 * M_PI * damping_frequency_) /
      (pow(10.0, damping_intensity_ / -10.0)));
    b1_ = 1.0 - a1_;
  }

  // Parameters from parameter server
  double sampling_frequency_;
  double damping_frequency_;
  double damping_intensity_;

  int divider_;

  // Filter Parameters
  double a1_;
  double b1_;
};

template <typename T>
class LowPassFilter : public filters::FilterBase<T>
{
public:
  LowPassFilter();

  ~LowPassFilter() override;

  bool configure() override;

  bool update(const T & data_in, T & data_out) override;

private:
  void initializeStorage();

  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::unique_ptr<LowPassParameters> parameters_;

  // Callback for updating dynamic parameters
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_callback_handle_;

  // Storage for filtered message history
  Eigen::Matrix<double, Eigen::Dynamic, 1> msg_filtered, msg_filtered_old, msg_old;

  // Flag for if dynamic storage variables have been initialized to the correct length for message type T.
  std::atomic_bool storage_initialized_;
};

template <typename T>
LowPassFilter<T>::LowPassFilter() : storage_initialized_{ false }
{
}

template <typename T>
LowPassFilter<T>::~LowPassFilter()
{
}

template <typename T>
bool LowPassFilter<T>::configure()
{
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  logger_.reset(
    new rclcpp::Logger(this->logging_interface_->get_logger().get_child(this->filter_name_)));
  parameters_.reset(new LowPassParameters(this->param_prefix_));

  parameters_->initialize(this->params_interface_, logger_->get_name());

  parameters_->declare_parameters();

  if (!parameters_->get_parameters())
  {
    return false;
  }

  // Initialize storage vectors
  initializeStorage();

  // Add callback to dynamically update parameters
  on_set_callback_handle_ = this->params_interface_->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return parameters_->set_parameter_callback(parameters);
    });

  return true;
}

template<typename T>
inline void LowPassFilter<T>::initializeStorage()
{
  // NOTE: assumes that T is a scalar type
  msg_filtered = Eigen::Matrix<T, 1, 1>::Zero();
  msg_filtered_old = Eigen::Matrix<T, 1, 1>::Zero();
  msg_old = Eigen::Matrix<T, 1, 1>::Zero();
  storage_initialized_ = true;
}

template<>
inline void LowPassFilter<geometry_msgs::msg::WrenchStamped>::initializeStorage()
{
  // WrenchStamped messages have 6 fields
  msg_filtered = Eigen::Matrix<double, 6, 1>::Zero();
  msg_filtered_old = Eigen::Matrix<double, 6, 1>::Zero();
  msg_old = Eigen::Matrix<double, 6, 1>::Zero();
  storage_initialized_ = true;
}

template<>
inline void LowPassFilter<trajectory_msgs::msg::JointTrajectoryPoint>::initializeStorage()
{
  // noop for trajectory_msgs::msg::JointTrajectoryPoint, because the size of the vector needs to be calculated
  // from the actual size of the effort, position, velocity, and acceleration fields in the received messages.
}

template <>
inline bool LowPassFilter<geometry_msgs::msg::WrenchStamped>::update(
  const geometry_msgs::msg::WrenchStamped & data_in, geometry_msgs::msg::WrenchStamped & data_out)
{
  if (!this->configured_)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 2000, "Filter is not configured");
    return false;
  }

  parameters_->update();

  // IIR Filter
  msg_filtered = parameters_->b1_ * msg_old + parameters_->a1_ * msg_filtered_old;
  msg_filtered_old = msg_filtered;

  // TODO(destogl): use wrenchMsgToEigen
  msg_old[0] = data_in.wrench.force.x;
  msg_old[1] = data_in.wrench.force.y;
  msg_old[2] = data_in.wrench.force.z;
  msg_old[3] = data_in.wrench.torque.x;
  msg_old[4] = data_in.wrench.torque.y;
  msg_old[5] = data_in.wrench.torque.z;

  data_out.wrench.force.x = msg_filtered[0];
  data_out.wrench.force.y = msg_filtered[1];
  data_out.wrench.force.z = msg_filtered[2];
  data_out.wrench.torque.x = msg_filtered[3];
  data_out.wrench.torque.y = msg_filtered[4];
  data_out.wrench.torque.z = msg_filtered[5];
  data_out.header = data_in.header;
  return true;
}

template <>
inline bool LowPassFilter<trajectory_msgs::msg::JointTrajectoryPoint>::update(
  const trajectory_msgs::msg::JointTrajectoryPoint & data_in, trajectory_msgs::msg::JointTrajectoryPoint & data_out)
{
  if (!this->configured_)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 2000, "Filter is not configured");
    return false;
  }

  // If storage has not been initialized yet, do so at this point using the sizes of the fields in data_in
  if (!storage_initialized_)
  {
    const std::size_t n_rows_total = data_in.effort.size() + data_in.positions.size() + data_in.velocities.size() + data_in.accelerations.size();
    msg_filtered = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(n_rows_total, 1);
    msg_filtered_old = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(n_rows_total, 1);
    msg_old = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(n_rows_total, 1);
    storage_initialized_ = true;
  }

  data_out = data_in;

  parameters_->update();

  // IIR Filter
  msg_filtered = parameters_->b1_ * msg_old + parameters_->a1_ * msg_filtered_old;
  msg_filtered_old = msg_filtered;

  std::size_t index = 0;

  for (std::size_t i = 0; i < data_in.effort.size(); ++i)
  {
    msg_old[index] = data_in.effort[i];
    data_out.effort[i] = msg_filtered[index];
    ++index;
  }

  for (std::size_t i = 0; i < data_in.positions.size(); ++i)
  {
    msg_old[index] = data_in.positions[i];
    data_out.positions[i] = msg_filtered[index];
    ++index;
  }

  for (std::size_t i = 0; i < data_in.velocities.size(); ++i)
  {
    msg_old[index] = data_in.velocities[i];
    data_out.velocities[i] = msg_filtered[index];
    ++index;
  }

  for (std::size_t i = 0; i < data_in.accelerations.size(); ++i)
  {
    msg_old[index] = data_in.accelerations[i];
    data_out.accelerations[i] = msg_filtered[index];
    ++index;
  }

  return true;
}

template <typename T>
bool LowPassFilter<T>::update(const T & data_in, T & data_out)
{
  if (!this->configured_)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 2000, "Filter is not configured");
    return false;
  }

  if (!this->storage_initialized_)
  {
    RCLCPP_ERROR_SKIPFIRST_THROTTLE((*logger_), *clock_, 2000, "Storage not initialized");
    return false;
  }

  parameters_->update();

  // Filter  
  msg_filtered = parameters_->b1_ * msg_old + parameters_->a1_ * msg_filtered_old;
  msg_filtered_old = msg_filtered;
  msg_old = Eigen::Matrix<T, 1, 1>(data_in);  // NOTE: assumes that data_in is a scalar type

  return true;
}

}  // namespace control_filters

#endif  // CONTROL_FILTERS__LOW_PASS_FILTER_HPP_
